#!/bin/bash

if (( $# != 1 )) ; then
	echo "Usage $0 img_name"
	exit 1
fi

img_name=$(basename $1)
img_idx=$(echo ${img_name} | cut -d_ -sf3 | cut -d. -sf1)
img_len=$(ls -l $1 | awk '{print $5}')
img_ver=$(head -n 2 $1 | tail -n 1 | sed -e 's/.\+626E\(.\{4\}\).*/\1/')
fota_img_name="fw${img_idx}_v${img_ver}_s${img_len}.bin"
img_header_name=$(tempfile)

echo "\$1 = $1"
echo "img_name = ${img_name}"
echo "img_idx = ${img_idx}"
echo "img_len = ${img_len}"
echo "img_ver = ${img_ver}"
echo "fota_img_name = ${fota_img_name}"
echo "img_header_name = ${img_header_name}"

echo -n -e "fw${img_idx}:${img_ver}:${img_len}\r\n" > ${img_header_name}
cat ${img_header_name} $1 > ${fota_img_name}
rm -f ${img_header_name}
