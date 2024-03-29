#!/bin/sh
src_template_path=src/experiments/$1.argos
generated_experiment_name=torus_$1_$2_$3_$4_$5
log_file_path=src/experiments/generated_insectbot_torus/logs/$generated_experiment_name.log
output_experiment_path=src/experiments/generated_insectbot_torus/$generated_experiment_name.argos
python3 src/scripts/generate_insectbot_torus_experiment.py "$2" "$3" "$4" "$5" "$src_template_path" "$output_experiment_path"
if [ "$?" -eq "0" ]; then
argos3 -c "$output_experiment_path"
fi