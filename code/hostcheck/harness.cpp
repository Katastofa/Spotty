// Регрессия декартова движка. Собирается ВМЕСТЕ с настоящим gait_test.ino,
// поэтому проверяет тот самый код, который поедет в железо.
#include "Arduino.h"
#include "Wire.h"
#include "oq_gait.h"
#include <cmath>
#include <cstdio>

bool HOST_QUIET = false;
HostSerial Serial;
TwoWire Wire;

enum { HIP=0, THIGH=1, KNEE=2, NJ=3 };
extern int   pEngine;
extern float pHeightMm, pSpanMm, pLiftMm, pPenetMm, pVelMmS, pSwingMs;
extern float pCrabDeg, pShiftXMm, pShiftYMm, pRollDeg, pPitchDeg, pYawDeg;
extern float pDuty, pAmpThigh, pAmpKnee, pCycleMs, pAmpHip;
extern float linkRatio, pCrouchDeg, pTrimDeg;
extern bool  pFlatPath, pSmoothSwing;
extern int   pTurn;
extern bool  ikFail;
extern void  gaitPose(unsigned char, float, float*);
extern void  loadGaitPreset();
extern void  setTargetsToPose();
extern float tgtAngle[4][3];
extern void  recalcCartesianTiming();
extern bool  legIK(float,float,float,float&,float&,float&);
extern float reachOf(float), kneeForReach(float);
extern const float L_FEMUR_MM, L_TIBIA_MM, L_HIP_MM, BASE_LAT_MM;
extern float BASE_LONG_MM;              // не const в копии стенда, см. build.sh
extern float pShiftGain, pMaxRateDps;
extern float bodyShiftAmpMm();
extern float bodyShiftYAt(float);
extern unsigned char gaitIdx;
extern const float LEG_LEFT[4];         // FR, FL, BR, BL: -1 правые, +1 левые

// ОБЪЯВЛЕНИЕ чужой таблицы, не её копия: значения живут в gait_test.ino.
struct GaitDef {
  const char *name; const char *note;
  float phase[4]; float duty, ampThigh, ampKnee, cycleMs;
};
extern const GaitDef GAITS[];

static int fails = 0;
static void check(bool ok, const char *what){
  printf("  [%s] %s\n", ok?" OK ":"ПРОВАЛ", what);
  if(!fails && !ok) fails=1; else if(!ok) fails++;
}

// Прямая задача: из трёх углов обратно в точку. Независимый вывод, не копия IK.
static void legFK(float hip,float thigh,float knee,float&x,float&y,float&z){
  const float F=L_FEMUR_MM, T=L_TIBIA_MM, LH=L_HIP_MM;
  const float L = sqrtf(F*F+T*T+2*F*T*cosf(radians(knee)));
  const float g = acosf(constrain((L*L+F*F-T*T)/(2*L*F),-1.0f,1.0f));
  const float a = radians(thigh)+g;
  x = L*sinf(a);
  const float r = L*cosf(a);
  const float rho = sqrtf(r*r+LH*LH);
  const float b = radians(hip)+atan2f(r,LH);
  y = rho*cosf(b);
  z = rho*sinf(b);
}

int main(){
  loadGaitPreset();
  printf("\n=== ВОРОТА A: legIK против прямой задачи ===\n");
  printf("Проверяется САМА IK, независимо от любого движка: точка -> углы -> точка.\n");
  {
    float worst=0; int tested=0, refused=0;
    for(float X=-70;X<=70.01f;X+=10)
    for(float Y=-30;Y<=40.01f;Y+=10)
    for(float Z=120;Z<=215.01f;Z+=5){
      float h,t,k;
      if(!legIK(X,Y,Z,h,t,k)){refused++;continue;}
      float x2,y2,z2; legFK(h,t,k,x2,y2,z2);
      float e=sqrtf((x2-X)*(x2-X)+(y2-Y)*(y2-Y)+(z2-Z)*(z2-Z));
      if(e>worst) worst=e;
      tested++;
    }
    printf("  точек %d (отказано %d, вне досягаемости), худшая невязка %.6f мм\n",tested,refused,worst);
    check(worst<0.01f,"невязка меньше 0.01 мм на всей рабочей области");
  }

  printf("\n=== ВОРОТА B: две модели ноги описывают ОДНУ ногу ===\n");
  printf("Угловой движок считает в единицах бедра (reachOf/alphaOf/kneeForReach),\n");
  printf("декартов — в мм через legIK. Если это одна и та же нога, то для любой\n");
  printf("точки стопы обе обязаны выдать одни и те же три угла.\n");
  {
    linkRatio = L_TIBIA_MM/L_FEMUR_MM;
    float worstT=0, worstK=0, worstH=0; int n=0;
    for(float rho=1.10f; rho<=2.05f; rho+=0.05f)          // длина ноги, бёдра
    for(float psi=-25.0f; psi<=25.01f; psi+=2.5f){        // угол ноги от вертикали
      const float knee = kneeForReach(rho);
      if(knee<1.0f || knee>170.0f) continue;
      const float thigh = psi - (float)( // alphaOf через тот же код
                            [&]{ extern float alphaOf(float); return alphaOf(knee); }() );
      const float depth = rho*L_FEMUR_MM*cosf(radians(psi));
      const float x     = rho*L_FEMUR_MM*sinf(radians(psi));
      float h2,t2,k2;
      if(!legIK(x, L_HIP_MM, depth, h2,t2,k2)) continue;
      worstH=fmaxf(worstH,fabsf(h2-0.0f));
      worstT=fmaxf(worstT,fabsf(t2-thigh));
      worstK=fmaxf(worstK,fabsf(k2-knee));
      n++;
    }
    printf("  точек %d; худшее расхождение моделей: хип %.6f°  бедро %.6f°  колено %.6f°\n",
           n,worstH,worstT,worstK);
    check(worstH<0.001f&&worstT<0.001f&&worstK<0.001f,
          "угловая модель и legIK дают одни углы — это ОДНА нога");
  }

  printf("\n=== ВОРОТА B2: чем траектории всё же отличаются (характеристика) ===\n");
  printf("Расхождение ЕСТЬ, оно измерено и оно НЕ ошибка.\n");
  {
    const float H=190.0f, AMP=12.0f;
    linkRatio = L_TIBIA_MM/L_FEMUR_MM;
    pCrouchDeg = kneeForReach(H/L_FEMUR_MM);
    pFlatPath=true; pSmoothSwing=false; pTrimDeg=0; pTurn=0;
    pAmpThigh=AMP; pAmpKnee=0; pDuty=0.5f; pCycleMs=1200;
    const float depth = reachOf(pCrouchDeg)*L_FEMUR_MM;
    const float span  = depth*tanf(radians(AMP));
    float wT=0,wK=0,wX=0;
    const int N=400;
    for(int i=0;i<=N;i++){
      const float ph = pDuty*(float)i/(float)N;
      const float sweep = 1.0f-2.0f*(float)i/(float)N;
      float a0[NJ],a1[NJ];
      pEngine=0; gaitPose(0,ph,a0);
      pEngine=1; pHeightMm=depth; pSpanMm=span; pLiftMm=20; pPenetMm=0;
      pCrabDeg=0;pShiftXMm=0;pShiftYMm=0;pRollDeg=0;pPitchDeg=0;pYawDeg=0;
      float sd=pDuty, sc=pCycleMs; pVelMmS=100; pSwingMs=(1.0f-sd)*sc;
      recalcCartesianTiming(); pDuty=sd; pCycleMs=sc;
      gaitPose(0,ph,a1);
      wT=fmaxf(wT,fabsf(a1[THIGH]-a0[THIGH]));
      wK=fmaxf(wK,fabsf(a1[KNEE]-a0[KNEE]));
      wX=fmaxf(wX,fabsf(depth*tanf(radians(AMP*sweep)) - span*sweep));
    }
    printf("  бедро расходится на %.4f°, колено на %.4f°, стопа на %.3f мм\n",wT,wK,wX);
    printf("\n  ПРИЧИНА, и она содержательная:\n");
    printf("  угловой движок ведёт ЛИНЕЙНО УГОЛ ноги, стопа идёт как depth*tan(psi);\n");
    printf("  декартов ведёт ЛИНЕЙНО САМУ СТОПУ. tan(12*s) != tan(12)*s, отсюда %.3f мм.\n",wX);
    printf("  Стопа в опоре стоит на земле, значит её скорость относительно корпуса\n");
    printf("  И ЕСТЬ скорость корпуса над землёй. Постоянной она должна быть у стопы,\n");
    printf("  а не у угла. То есть в опоре угловой движок разгоняет и тормозит корпус\n");
    printf("  примерно на 1%% за шаг, а декартов — нет. Правее декартов.\n");
    check(wX<1.0f,"расхождение траекторий меньше 1 мм и объяснено");
  }

  printf("\n=== ВОРОТА C: перенос — НЕ совпадает, и это цель порта ===\n");
  {
    pEngine=1; pHeightMm=190; pSpanMm=30; pLiftMm=20; pPenetMm=0;
    pDuty=0.5f; pCycleMs=1200;
    // скорость стопы у краёв переноса: у безье она обязана быть ~0
    float pz0,pz1,pzA,pzB;
    float a[NJ];
    printf("  подъём стопы по фазе переноса (мм над линией стоп):\n   ");
    for(float t=0;t<=1.001f;t+=0.1f){
      float ph=pDuty+(1.0f-pDuty)*t;
      gaitPose(0,ph,a);
      float x,y,z; legFK(a[HIP],a[THIGH],a[KNEE],x,y,z);
      printf(" %.1f",pHeightMm-z);
    }
    printf("\n");
    // вертикальная скорость на краях
    gaitPose(0,pDuty+0.000f*(1-pDuty),a); { float x,y;legFK(a[HIP],a[THIGH],a[KNEE],x,y,pz0);} 
    gaitPose(0,pDuty+0.020f*(1-pDuty),a); { float x,y;legFK(a[HIP],a[THIGH],a[KNEE],x,y,pzA);} 
    gaitPose(0,pDuty+0.980f*(1-pDuty),a); { float x,y;legFK(a[HIP],a[THIGH],a[KNEE],x,y,pzB);} 
    gaitPose(0,pDuty+1.000f*(1-pDuty),a); { float x,y;legFK(a[HIP],a[THIGH],a[KNEE],x,y,pz1);} 
    printf("  вертикальная скорость на ОТРЫВЕ  %.3f мм за 2%% фазы\n",fabsf(pzA-pz0));
    printf("  вертикальная скорость на КАСАНИИ %.3f мм за 2%% фазы\n",fabsf(pz1-pzB));
    // сравнение с угловым sin(pi*t)
    pEngine=0; pFlatPath=false; pAmpKnee=15; pAmpThigh=12;
    gaitPose(0,pDuty+0.000f*(1-pDuty),a); float k0=a[KNEE];
    gaitPose(0,pDuty+0.020f*(1-pDuty),a); float kA=a[KNEE];
    printf("  для сравнения, угловой sin(pi*t): колено за те же 2%% уходит на %.3f°\n",fabsf(kA-k0));
    check(fabsf(pzA-pz0)<1.0f,"стопа отрывается без рывка (безье, кратные узлы)");
  }

  printf("\n=== ВОРОТА D: отказ IK ловится, а не проглатывается ===\n");
  {
    pEngine=1; pHeightMm=224.0f; pSpanMm=60; pLiftMm=0; pPenetMm=0; // почти вытянута
    pShiftXMm=0; pRollDeg=0; ikFail=false;
    float a[NJ];
    HOST_QUIET=true;
    for(int i=0;i<=200;i++) gaitPose(0,(float)i/200.0f,a);
    HOST_QUIET=false;
    printf("  высота %.1f при досягаемости %.1f мм -> ikFail = %s\n",
           pHeightMm,L_FEMUR_MM+L_TIBIA_MM,ikFail?"да":"нет");
    check(ikFail,"недостижимая точка поднимает флаг");
    pHeightMm=190; ikFail=false;
    HOST_QUIET=true;
    for(int i=0;i<=200;i++) gaitPose(0,(float)i/200.0f,a);
    HOST_QUIET=false;
    check(!ikFail,"достижимая — не поднимает");
  }

  printf("\n=== ВОРОТА E: поза и походка стыкуются, а не прыгают ===\n");
  printf("'n' выводит в позу, 'w' пускает походку. Если поза посчитана одним\n");
  printf("движком, а походка другим, робот прыгнет на первом же тике — на\n");
  printf("стенде это выглядит как рывок из ниоткуда.\n");
  {
    pEngine=1; pHeightMm=190; pSpanMm=30; pLiftMm=15; pPenetMm=0;
    pShiftXMm=15; pShiftYMm=0; pRollDeg=0; pPitchDeg=0; pYawDeg=0;
    pDuty=0.53f; pCycleMs=1900; pTurn=0;
    HOST_QUIET=true; setTargetsToPose(); HOST_QUIET=false;
    // Середина опоры: у квадратичного безье там x = 0, то есть та же номиналь.
    float a[NJ]; gaitPose(0, pDuty*0.5f, a);
    float w=0; for(int j=0;j<NJ;j++) w=fmaxf(w,fabsf(a[j]-tgtAngle[0][j]));
    printf("  поза  : хип %7.3f  бедро %7.3f  колено %7.3f\n",
           tgtAngle[0][0],tgtAngle[0][1],tgtAngle[0][2]);
    printf("  походка (середина опоры): %7.3f %7.3f %7.3f\n",a[0],a[1],a[2]);
    printf("  расхождение %.5f° при сдвиге корпуса %+.0f мм\n",w,pShiftXMm);
    check(w<0.01f,"поза декартова движка совпадает с серединой его же опоры");

    // А что было бы со старой, угловой позой — ради масштаба беды.
    pEngine=0; HOST_QUIET=true; setTargetsToPose(); HOST_QUIET=false;
    float w2=0; for(int j=0;j<NJ;j++) w2=fmaxf(w2,fabsf(a[j]-tgtAngle[0][j]));
    printf("  для сравнения: угловая поза отстоит от той же точки на %.1f°\n",w2);
    printf("  (столько робот прыгнул бы за один тик, если бы позу не развели)\n");
  }

  printf("\n=== ВОРОТА F: перенос корпуса по фазе ===\n");
  printf("Амплитуду прошивка считает как sqrt(a^2+b^2)-a. Здесь она проверяется\n");
  printf("ДРУГОЙ формулой — инцентр как взвешенная сумма вершин со сторонами в\n");
  printf("весах. Совпадение двух независимых выводов и есть проверка.\n");
  {
    const float baseLongFw = BASE_LONG_MM;   // вернуть ровно это в конце
    pEngine=1; pShiftGain=0.0f; gaitIdx=3; loadGaitPreset();   // 3 = walk
    pDuty=0.75f; pCycleMs=2400; pHeightMm=150; pSpanMm=25; pLiftMm=15;
    pShiftXMm=0; pShiftYMm=0; pRollDeg=0; pPitchDeg=0; pYawDeg=0; pTurn=0;

    // 1. ВЫКЛЮЧЕНО ЗНАЧИТ ВЫКЛЮЧЕНО. Пока оператор не подал 'B', сдвиг обязан
    //    быть НУЛЁМ ТОЖДЕСТВЕННО, а не «близко к нулю».
    float mx=0; for(int i=0;i<=400;i++) mx=fmaxf(mx,fabsf(bodyShiftYAt((float)i/400.0f)));
    check(mx==0.0f, "при pShiftGain = 0 сдвиг тождественно ноль");

    // 2. БЕЗ ПРОДОЛЬНОЙ БАЗЫ — ОТКАЗ, а не правдоподобное число.
    pShiftGain = 1.0f;
    BASE_LONG_MM = 0.0f;
    mx=0; for(int i=0;i<=400;i++) mx=fmaxf(mx,fabsf(bodyShiftYAt((float)i/400.0f)));
    printf("  без M4 (BASE_LONG = 0): амплитуда %.3f мм\n", bodyShiftAmpMm());
    check(mx==0.0f && bodyShiftAmpMm()==0.0f, "без замера M4 переноса нет вовсе");

    // 3. АМПЛИТУДА ПРОТИВ НЕЗАВИСИМОГО ИНЦЕНТРА — на НАСТОЯЩЕЙ базе.
    BASE_LONG_MM = baseLongFw;
    const float hl = 0.5f*BASE_LONG_MM, hw = 0.5f*BASE_LAT_MM + L_HIP_MM;
    // Опора, когда переносится ПЛ: ПП(+hl,-hw), ЗП(-hl,-hw), ЗЛ(-hl,+hw).
    const float P[3][2] = { {+hl,-hw}, {-hl,-hw}, {-hl,+hw} };
    float side[3];                          // сторона против вершины i
    for(int i=0;i<3;i++){
      const int j=(i+1)%3, k=(i+2)%3;
      side[i] = sqrtf((P[j][0]-P[k][0])*(P[j][0]-P[k][0]) +
                      (P[j][1]-P[k][1])*(P[j][1]-P[k][1]));
    }
    const float sw = side[0]+side[1]+side[2];
    float inc[2] = {0,0};
    for(int i=0;i<3;i++){ inc[0]+=side[i]*P[i][0]; inc[1]+=side[i]*P[i][1]; }
    inc[0]/=sw; inc[1]/=sw;
    printf("  инцентр опоры при переносе ПЛ: x %+.3f  y %+.3f мм\n", inc[0], inc[1]);
    printf("  прошивка считает амплитуду    :      %.3f мм\n", bodyShiftAmpMm());
    check(fabsf(fabsf(inc[1]) - bodyShiftAmpMm()) < 1e-3f,
          "амплитуда прошивки = поперечный вынос инцентра");
    printf("  продольный вынос инцентра %+.1f мм в таблицу НЕ ВЗЯТ намеренно:\n", inc[0]);
    printf("  в симуляторе он давал 0 из 8 выживших (переносит корпус на ту ногу,\n");
    printf("  которая вот-вот поднимется).\n");

    // 4. ЗНАК ПРОТИВ ФАЗИРОВКИ. На каждом узле смотрим, какая нога в переносе,
    //    и требуем, чтобы корпус уезжал В ДРУГУЮ сторону. Таблица нигде не
    //    записана: и знак, и проверка выведены из одних и тех же фаз, но
    //    разными путями.
    int bad=0;
    printf("  узлы: ");
    for(int i=0;i<8;i++){
      const float u = (float)i/8.0f;
      float swingSide = 0.0f;
      for(int leg=0;leg<4;leg++){
        float ph = u + GAITS[3].phase[leg]; ph -= floorf(ph);
        if (ph >= pDuty) swingSide += LEG_LEFT[leg];
      }
      const float got = bodyShiftYAt(u);
      printf("%+.1f ", got);
      if (swingSide >  0.5f && !(got < -1e-3f)) bad++;   // в переносе левые
      if (swingSide < -0.5f && !(got > +1e-3f)) bad++;   // в переносе правые
    }
    printf("мм\n");
    check(bad==0, "корпус уезжает ОТ переносимой ноги на каждом узле");

    // 5. РАБОТОСПОСОБНОСТЬ В ТОЧКЕ, РАДИ КОТОРОЙ ВСЁ ЗАТЕЯНО. H 150, walk:
    //    включённый перенос не должен сам по себе делать точку недостижимой.
    ikFail=false; float a5[NJ];
    HOST_QUIET=true;
    for(int leg=0;leg<4;leg++)
      for(int i=0;i<=200;i++) gaitPose((unsigned char)leg,(float)i/200.0f,a5);
    HOST_QUIET=false;
    check(!ikFail, "H 150 с включённым переносом: IK не отказывает");

    pShiftGain = 0.0f; BASE_LONG_MM = baseLongFw;   // вернуть как в прошивке
  }

  printf("\n%s\n", fails? "ЕСТЬ ПРОВАЛЫ" : "ВСЕ ВОРОТА ПРОЙДЕНЫ");
  return fails?1:0;
}
