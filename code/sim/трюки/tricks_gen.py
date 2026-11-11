"""Пересобрать code/gait_test/spotty_tricks.h из tricks.py."""
from tricks import TRICKS
NOTE = {
 'hiptest':'корпус влево на 15 мм и обратно. На подвесе: стопы должны уехать ВПРАВО',
 'sit':"сесть на круп и остаться сидеть ('n' — встать)",
 'paw':'сесть, дать правую переднюю лапу, встать',
 'wave':'сесть, помахать правой передней лапой, встать',
 'bow':'поклон: нос вниз вокруг задних стоп',
 'stretch':'потянуться: нос вниз, потом нос вверх',
 'pushup':'отжимания передом, 3 раза',
 'dance':'танец: корпус по кругу, стопы на месте',
 'wiggle':'вилять задом',
 'look':'оглядеться влево-вправо',
}
out=[]
out.append('// --- СГЕНЕРИРОВАНО code/sim/трюки/tricks.py -> tricks_gen.py. Правка — там, не здесь. ---')
names=[]
for key,(ru,keys,sit) in TRICKS.items():
    hip = any(abs(d[1])>1e-6 for _,k in keys for d in k) and key!='hiptest'
    out.append('static const TrickKey TRK_%s[] = {' % key.upper())
    for ms,k in keys:
        out.append('  { %4d, {%s} },' % (ms, ', '.join('{%d,%d,%d}' % tuple(int(round(v*10)) for v in d) for d in k)))
    out.append('};')
    names.append('  { "%s", "%s", "%s", TRK_%s, sizeof(TRK_%s)/sizeof(TRK_%s[0]), %s, %s },' % (
        key, ru, NOTE[key], key.upper(), key.upper(), key.upper(), 'true' if sit else 'false', 'true' if hip else 'false'))
out.append('static const TrickDef TRICKS[] = {')
out+=names
out.append('};')
import os
dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'gait_test', 'spotty_tricks.h')
with open(dst, 'w', encoding='utf-8', newline='\n') as f:
    f.write('#pragma once\n#include "spotty_tricks_types.h"\n' + '\n'.join(out) + '\n')
print('записано', os.path.normpath(dst))

