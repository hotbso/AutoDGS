RESDIR=/e/X-Plane-12-test/Resources/plugins/AutoDGS/resources
if [ -d $RESDIR ]
then
    for f in *
    do
        [[ $f == *-base.obj ]] && continue
        [[ $f == *.obj || $f == *.png || $f == *.dds ]] && cp -p $f $RESDIR/.
    done
fi
