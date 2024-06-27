#!/bin/bash

#check if version is passed otherwise use current date
if [ $# -eq 0 ]
  then
    version=$(date +"%Y.%m.%d")
  else
    version=$1
fi

#update versions in control file
sed -i -e 's/^Version: .*/Version: '"${version}"'/' package/DEBIAN/control

#move all python files to correct location
mkdir -p package/usr/lib/python3/dist-packages
cp -r  build/lib/* package/usr/lib/python3/dist-packages
#move all library files to correct location
mkdir -p package/usr/lib
cp -r build/python_packaging/install/lib/*\.so* package/usr/lib
mkdir -p package/usr/include
cp -r  build/python_packaging/install/include/* package/usr/include

#package deb
dpkg-deb --root-owner-group -b package provizio-dds-"$version"_arm64.deb
