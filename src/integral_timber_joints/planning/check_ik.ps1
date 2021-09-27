# Shelter process has seq_n 0 - 56
# Set-Location C:\Users\leungp\Documents\GitHub\integral_timber_joints

for ($num = 21 ; $num -le 56 ; $num++) {
    python -m integral_timber_joints.planning.check_ik --design_dir 210419_AnticlasticShelter --problem shelter_process.json --seq_n $num
}