#!/bin/bash -l
####### RESOURCES SETUP #######
#These commands set up the Grid Environment for your job:
# -N [Job Name]
# -l nodes=[number of cores]
# -l ncpus=[number of cpus required]
# -l mem=[amount of memory required]
# -l walltime=[how long the job can run for]
# -m ae = mail on (a)bort and (e)rror
# -o $PBS_O_WORKDIR/[stdout file name]
# -e $PBS_O_WORKDIR/[stderr output file name]

### Select Resouces
#PBS -N sys_single
#PBS -l nodes=1:ppn=1
#PBS -l mem=4000mb
#PBS -l walltime=02:00:00

### Output File
#PBS -j oe
#PBS -o run_output_single_system.txt

### Queue
#pbs -q workq

### Mail Options
#PBS -m aeb

####### START PROGRAM #######
echo "Load GCC"      && module load gcc
echo "Load cmake"    && module load cmake
echo "Load openmpi"  && module load openmpi
echo "Load mpi"      && module load mpi
echo "Load Intel"    && module load intel
echo "Load scotch"   && module load scotch
echo "Load paraview" && module load paraview
echo "Load openfoam" && module load openfoam

# If testing in local session
if [ -z "${PBS_O_HOME+xxx}" -o -z "${PBS_O_WORKDIR+xxx}" ]; then
    echo "Running on a local machine (PBS variables not set)"
    export PBS_O_HOME=$HOME
    export PBS_O_WORKDIR=`pwd`
fi

# Change to our working directory
cd $PBS_O_WORKDIR

# Start timing
START=$(date +%s)

# Run Case
echo "Running Case"
echo "Load Case Functions" && source case_functions.sh
cleanCase
runSingle


# Get runtime
END=$(date +%s)
DIFF=$(( $END - $START ))
echo "Runtime was $DIFF seconds" > time_pbs_single
