rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../src
make
sudo make install

cd ..
echo "Build is done!"