from pathlib import Path

p = Path("control/patch_dashboard.py")
s = p.read_text(encoding="utf-8")

old = '''replace_once(
    '        self.wait_count = 0\\n',
    \\'\\'\\'        self.wait_count = 0
        self.api_client = None
\\'\\'\\',
    "API client state"
)'''

new = '''replace_once(
    "        self.process = None\\n",
    \\'\\'\\'        self.process = None
        self.api_client = None
\\'\\'\\',
    "API client state"
)'''

if old not in s:
    print("Could not find the expected patch-script section.")
    print("No changes made to the patch script.")
    raise SystemExit(1)

p.write_text(s.replace(old, new, 1), encoding="utf-8")
print("Patch script corrected.")
