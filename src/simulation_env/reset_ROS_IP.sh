#!/bin/bash
tag=$(cat ~/.bashrc | tail -n 1)
if [ $tag == "#LDSCtag" ];then
    echo $tag
    echo "reset"
    #sed -i '$d' ~/.bashrc
    sed -i '$d' ~/.bashrc
    sed -i '$d' ~/.bashrc    
else
    echo $tag
    echo "nothing to reset"
fi

