#!/bin/bash

f=`find . -name \*.html -or -name \*.js`
for file in $f
do
 iconv -f UTF8 -t CP1251 $file -o tmpfile && mv tmpfile $file
done