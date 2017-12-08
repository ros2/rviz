# copied from rviz_ogre_vendor/env_hook/rviz_ogre_vendor_library_path.sh

# detect if running on Darwin platform
_UNAME=`uname -s`
_IS_DARWIN=0
if [ "$_UNAME" = "Darwin" ]; then
  _IS_DARWIN=1
fi
unset _UNAME

if [ $_IS_DARWIN -eq 0 ]; then
  ament_prepend_unique_value LD_LIBRARY_PATH "$AMENT_CURRENT_PREFIX/opt/rviz_ogre_vendor/lib"
else
  ament_prepend_unique_value DYLD_LIBRARY_PATH "$AMENT_CURRENT_PREFIX/opt/rviz_ogre_vendor/lib"
fi
unset _IS_DARWIN
