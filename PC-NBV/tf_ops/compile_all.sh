cd /home/cuda/Alex/trai/PC-NBV/tf_ops

cd approxmatch

bash tf_approxmatch_compile.sh

echo "Approxmatch compile complete"

cd ..

cd grouping

bash tf_grouping_compile.sh

echo "Grouping compile complete"

cd ..

cd interpolation

bash tf_interpolate_compile.sh

echo "Interpolation compile comple"

cd ..

cd nn_distance

bash tf_nndistance_compile.sh

echo "NN Distance compile complete"

cd ..

cd renderball

bash compile_render_balls_so.sh

echo "Renderball compile complete"

cd ..

cd sampling

bash tf_sampling_compile.sh

echo "Sampling compile complete"

cd ..

