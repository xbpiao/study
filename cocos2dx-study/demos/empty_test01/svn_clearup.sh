# 清除proj.android
rm -f -r proj.android/libs
rm -f -r proj.android/bin
rm -f -r proj.android/obj
rm -f -r proj.android/assets
rm -f -r proj.android/gen
rm -f -r proj.android-studio/build
rm -f -r proj.android-studio/app/build
rm -f -r proj.android-studio/app/.externalNativeBuild
rm -f -r bin

# 清除proj.win32
rm -f -r proj.win32/Debug.win32
rm -f -r proj.win32/.vs

find . -name "*.bak" -exec rm -f {} \;
find . -name "*.o" -exec rm -f {} \;
find . -name "*.a" -exec rm -f {} \;
find . -name "*.sdf" -exec rm -f {} \;
find . -name "*.VC.db" -exec rm -f {} \;
find . -name "*.VC.opendb" -exec rm -f {} \;
find . -name "*.suo" -exec rm -f {} \;
find . -name ".DS_Store" -exec rm -f {} \;