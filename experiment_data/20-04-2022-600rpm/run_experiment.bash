for (( i=$1; i<=1000; i++ )); do
	if [[ $i -ne 1 ]]; then
		cp config/experiment$((i-1)).cfg config/experiment$i.cfg;
	fi;
	echo "Starting experiment #${i}...";
	echo "Press enter to continue...";
	read;
	mkdir results/experiment$i;
	../../build/algorithm_simulation --c config/experiment$i.cfg --i frames/processed/*.pcd --o results/experiment$i/ --threads 12;
	echo "#######################";
	echo "# EXPERIMENT COMPLETE #";
	echo "#######################";
	echo "";
done;
