#!/bin/sh

# this script will update Eigen to version 3.4.0
# tested on Ubuntu 20.04

# download Eigen 3.4.0 as .zip
printf "Download Eigen 3.4.0 as .zip . . . \n"
wget -O eigen-3.4.0.zip 'https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip'

# unzip
printf "Unzip file . . . \n"
unzip eigen-3.4.0.zip

# remove old Eigen
printf "Remove old Eigen headers in /usr/include/eigen3 . . . \n"
sudo rm -fr /usr/include/eigen3/Eigen
sudo rm -fr /usr/include/eigen3/unsupported
sudo rm /usr/include/eigen3/signature_of_eigen3_matrix_library

# move new version
printf "Move new Eigen headers to /usr/include/eigen3 . . . \n"
sudo mv eigen-3.4.0/Eigen /usr/include/eigen3/Eigen
sudo mv eigen-3.4.0/unsupported /usr/include/eigen3/unsupported
sudo mv eigen-3.4.0/signature_of_eigen3_matrix_library /usr/include/eigen3/signature_of_eigen3_matrix_library

# remove local Eigen data
printf "Remove local eigen-3.4.0 directory and eigen-3.4.0.zip . . . \n"
rm -fr eigen-3.4.0
rm eigen-3.4.0.zip

printf "Updated Eigen Headers in /usr/include/eigen3\n"