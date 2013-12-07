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
#PBS -N usr_par
#PBS -l nodes=1:ppn=4
#PBS -l mem=4000mb
#PBS -l walltime=02:00:00

### Output File
#PBS -j oe
#PBS -o run_output_parallel_user.txt

### Queue
#pbs -q workq

### Mail Options
#PBS -m aeb

####### START PROGRAM #######
echo "Load GCC" && module load gcc
echo "Load cmake"    && module load cmake
echo "Load openmpi" && module load openmpi
echo "Load mpi"      && module load mpi
echo "Load Intel" && module load intel
echo "Load scotch" && module load scotch
echo "Load paraview" && module load paraview
echo "Load openfoam local" && source $PBS_O_HOME/OpenFOAM/OpenFOAM-2.2.2/etc/bashrc
mpirun=/pkg/suse11/openmpi/1.6.5/bin/mpirun


if [ -z "${PBS_O_HOME+xxx}" -o -z "${PBS_O_WORKDIR+xxx}" ]; then
    echo "Running on a local machine (PBS variables not set)"
    export PBS_O_HOME=$HOME
    export PBS_O_WORKDIR=`pwd`
    export _NCPU=4
else
    echo "Running on PBS"
    export _NCPU=`sed -n '$=' $PBS_NODEFILE`
    mpirun="$mpirun --hostfile $PBS_NODEFILE -np $_NCPU"
fi

echo "Job has access to `nproc` threads per node, and will use $_NCPU"

# Change to our working directory
cd $PBS_O_WORKDIR

# Start timing
START=$(date +%s)

# Setup Decomposition
echo "
/*--------------------------------*- C++ -*----------------------------------*\
| =========                 |                                                 |
| \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox           |
|  \\    /   O peration     | Version:  2.2.2                                 |
|   \\  /    A nd           | Web:      www.OpenFOAM.org                      |
|    \\/     M anipulation  |                                                 |
\*---------------------------------------------------------------------------*/
FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    location    \"system\";
    object      decomposeParDict;
}
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
numberOfSubdomains $_NCPU;
method             scotch;" > system/decomposeParDict

# Run Case
echo "Running Case"
echo "Load Case Functions" && source caseFunctions.sh
cleanCase
runParallel


# Get runtime
END=$(date +%s)
DIFF=$(( $END - $START ))
echo "Runtime was $DIFF seconds" > time_pbs_single