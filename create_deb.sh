#!/bin/bash

version="2024.06.25"

#update versions in control file

#move all python files to correct location
mkdir -p package/usr/lib/python3/dist-packages
cp -r  build/lib/* package/usr/lib/python3/dist-packages
#move all library files to correct location
mkdir -p package/usr/lib
cp -r build/python_packaging/install/lib/*\.so* package/usr/lib
mkdir -p package/usr/include
cp -r  build/python_packaging/install/include/* package/usr/include

#create new package directory from template
cp -r package provizio-dds-$version

#package deb
dpkg-deb --root-owner-group -b provizio-dds-$version provizio-dds-"$version"_arm64.deb
