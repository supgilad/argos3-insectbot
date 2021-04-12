#!/bin/sh
src_template_path=src/examples/experiments/$1.argos
generated_experiment_name=torus_$1_$2_$3_$4_$5
output_experiment_path=src/examples/experiments/generated_insectbot_torus/$generated_experiment_name.argos
python3 generate_insectbot_torus_experiment.py "$2" "$3" "$4" "$5" "$src_template_path" "$output_experiment_path"
argos3 -l src/examples/experiments/generated_insectbot_torus/logs/$generated_experiment_name.log -c "$output_experiment_path"
