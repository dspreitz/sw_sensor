#!/usr/bin/env python3
"""Compile every source of the Release configuration in sw_stm32/.cproject with
the flags of that configuration and report the diagnostics (temporary check)."""
import os, re, subprocess, sys, xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor

ROOT = "sw_stm32"
cfg = next(c for c in ET.parse(f"{ROOT}/.cproject").iter("configuration") if c.get("name") == "Release")

def options(kind):
    tool = next(t for t in cfg.iter("tool") if t.get("superClass", "").endswith(f"tool.{kind}.compiler"))
    o = {}
    for opt in tool.iter("option"):
        key = opt.get("superClass").split(".")[-1]
        o[key] = (opt.get("value"), [l.get("value") for l in opt.iter("listOptionValue")])
    return o

def paths(vals):
    out = []
    for v in vals:
        v = v.strip('"')
        if v.startswith("${workspace_loc:/${ProjName}/"):
            v = v[len("${workspace_loc:/${ProjName}/"):].rstrip("}")
        elif v.startswith("../"):  # relative to the build directory sw_stm32/Release
            v = v[3:]
        out.append("-I" + ROOT + "/" + v)
    return out

def defines(vals):
    return ["-D" + v.replace(" ", "") for v in vals]

c, cpp = options("c"), options("cpp")
common = ["-mcpu=cortex-m4", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard", "-mthumb", "-O3", "-ffunction-sections", "-fdata-sections", "-Wall", "-fstack-usage"]
cflags = common + ["-std=gnu11"] + defines(c["definedsymbols"][1]) + paths(c["includepaths"][1])
cxxflags = common + ["-std=gnu++14", "-fno-exceptions", "-fno-rtti", "-fno-use-cxa-atexit"] + defines(cpp["definedsymbols"][1]) + paths(cpp["includepaths"][1])
for key, flag in [("extra", "-Wextra"), ("pedanticerrors", "-pedantic-errors"), ("missing_include_dirs", "-Wmissing-include-dirs"),
                  ("w_switch_default", "-Wswitch-default"), ("w_switch_enum", "-Wswitch-enum"), ("conversion", "-Wconversion")]:
    if cpp.get(key, ("false",))[0] == "true":
        cxxflags.append(flag)

sources = []
for e in cfg.iter("entry"):
    if e.get("kind") != "sourcePath":
        continue
    base = os.path.join(ROOT, e.get("name"))
    excl = [os.path.join(base, x) for x in (e.get("excluding") or "").split("|") if x]
    for d, _, files in os.walk(base):
        for f in files:
            p = os.path.join(d, f)
            if not f.endswith((".c", ".cpp")) or any(p == x or p.startswith(x + "/") for x in excl):
                continue
            sources.append(p)

def compile(p):
    cc = ["arm-none-eabi-g++"] + cxxflags if p.endswith(".cpp") else ["arm-none-eabi-gcc"] + cflags
    r = subprocess.run(cc + ["-c", p, "-o", "/dev/null"], capture_output=True, text=True)
    return p, r.returncode, r.stderr

with ThreadPoolExecutor(os.cpu_count()) as ex:
    results = sorted(ex.map(compile, sorted(sources)))
failed = [r for r in results if r[1]]
diags = {}
for p, rc, err in results:
    for m in re.finditer(r"^(\S+?):(\d+):\d+: (error|warning): (.*?)(?: \[(-W[^\]]+)\])?$", err, re.M):
        diags.setdefault((m.group(3), m.group(5) or "(no flag)"), []).append(f"{m.group(1)}:{m.group(2)}: {m.group(4)}")
print(subprocess.run(["arm-none-eabi-gcc", "--version"], capture_output=True, text=True).stdout.splitlines()[0])
print(f"{len(sources)} sources, {len(failed)} failed")
for (kind, flag), items in sorted(diags.items(), key=lambda kv: (kv[0][0] != "error", -len(kv[1]))):
    uniq = sorted(set(items))
    print(f"\n{kind} {flag}: {len(uniq)}")
    for i in uniq[: (50 if kind == "error" else 5)]:
        print("   ", i)
for p, rc, err in failed:
    print(f"\n=== FAILED {p}\n{err[-1500:]}")
