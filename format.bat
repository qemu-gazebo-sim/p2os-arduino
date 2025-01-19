@echo off

for /R src %%f in (*.cpp *.h *.hpp) do clang-format -i "%%f"
for /R examples %%f in (*.cpp *.h *.hpp) do clang-format -i "%%f"

echo Done!