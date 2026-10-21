#!/bin/bash
#SBATCH --account=nn9970K
#SBATCH --job-name=copepod_1px
#SBATCH --time=02:00:00          # 2 hours (copepod is smaller than MNIST, should be faster)
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=52
#SBATCH --mem=160G
#SBATCH --partition=normal

#SBATCH --output=copepod_log_%j.txt
#SBATCH --error=copepod_err_%j.txt

set -o errexit
set -o nounset

module purge
module load intel/2021a
module load MATLAB/2022b

export MLM_LICENSE_FILE=8655@matlab.lisens.ntnu.no

# TIRA is single-threaded per worker (confirmed via htop on MNIST runs)
# 48 workers × 1 thread = 48 cores. Remaining 4 for master + OS.
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1

export SLURM_JOB_TIME_LIMIT="02:00:00"

matlab -nodisplay -batch "run_copepod_quick"
