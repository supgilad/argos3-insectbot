rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../src
make
sudo make install

# change robot to cube
# create behaviour of not hitting walls
#
cd ..
argos3 -c src/examples/experiments/insectbot_avoider.argos 
