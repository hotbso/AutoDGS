#/bin/bash
LACFU_MASTER=../libacfutils/src

if [ ! -d "$LACFU_MASTER" ]
then
    echo "$LACFU_MASTER not found" >2
    exit 1
fi

function update_dir() {
    for f in $1
    do
        b=$(basename "$f")
        if [[ "$b" =~ ^airportdb\..* ]]
        then
            echo "skip airportdb.*"
            continue
        fi

        if [[ "$b" == conf.c ]]
        then
            echo "skip conf.c"
            continue
        fi

        echo "cp -p $2/$b $f"
        cp -p $2/$b $f
    done
}

update_dir "libacfutils/src/*.c" $LACFU_MASTER
update_dir "libacfutils/src/acfutils/*.h" $LACFU_MASTER/acfutils