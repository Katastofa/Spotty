"""
Как Arduino IDE готовит .ino к компиляции — в той части, которая ломает сборку.

Arduino (arduino-cli / ctags) находит все функции скетча и вставляет их
ПРОТОТИПЫ перед ПЕРВОЙ функцией файла. Если в сигнатуре тип, объявленный в
.ino ниже этого места, сборка падает: "'X' does not name a type". Обычный g++
(build.sh, websyntax.sh) этого не видит — так 2026-09-29 проскочил TrickKey.

    python arduino_proto.py in.ino out.cpp

Приближение, не копия ctags: берутся однострочные определения функций с
колонки 0; значения по умолчанию из прототипа убираются.
"""
import re, sys

SKIP = {"if", "for", "while", "switch", "return", "else", "do", "struct", "class",
        "enum", "union", "typedef", "namespace", "static_assert", "template"}
RX = re.compile(r'^([A-Za-z_][\w:<>\*&\s]*?[\s\*&])([A-Za-z_]\w*)\s*\(([^;{}]*)\)\s*(const\s*)?\{')

def strip_defaults(params):
    out, depth, cur = [], 0, ""
    for ch in params:
        if ch in "([{<": depth += 1
        if ch in ")]}>": depth -= 1
        if ch == "," and depth == 0:
            out.append(cur); cur = ""
        else:
            cur += ch
    out.append(cur)
    return ",".join(p.split("=")[0].rstrip() for p in out)

def main(src, dst):
    lines = open(src, encoding="utf-8").read().split("\n")
    protos, first, depth, in_block_comment = [], None, 0, False
    for i, ln in enumerate(lines):
        code = re.sub(r'//.*', '', ln)
        if depth == 0 and not in_block_comment and not ln.startswith((" ", "\t", "#")):
            m = RX.match(code)
            prev = lines[i - 1].strip() if i else ""
            if m and m.group(1).split()[0] not in SKIP and not prev.startswith("template"):
                ret, name, params = m.group(1).strip(), m.group(2), m.group(3)
                protos.append("%s %s(%s);" % (ret, name, strip_defaults(params)))
                if first is None:
                    first = i
        # грубый учёт скобок и блочных комментариев
        s = re.sub(r'"(\\.|[^"\\])*"', '""', code)
        s = re.sub(r"'(\\.|[^'\\])'", "''", s)
        if "/*" in s and "*/" not in s: in_block_comment = True
        if in_block_comment and "*/" in s: in_block_comment = False
        if not in_block_comment:
            depth += s.count("{") - s.count("}")
    if first is None:
        first = 0
    out = lines[:first] + ["// --- прототипы, как их вставляет Arduino ---"] + protos + lines[first:]
    open(dst, "w", encoding="utf-8").write("\n".join(out))
    print("  прототипов вставлено: %d, перед строкой %d" % (len(protos), first + 1))

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
