set -x
set -e
cd ../initmem_gen2
./run-nuttx.sh
cd -
cp ../initmem_gen2/init_kernel.txt init_kernel.txt
make
./simv

