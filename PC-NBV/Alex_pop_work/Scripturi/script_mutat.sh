#! /bin/sh.

#read input1 
#read input2




path_valid="/media/rambo/external/test/Box_0/"






for i in ./*/;do	

	if [ -d "$i" ]; then
		cd "$i";
		echo $i;
	
		
		num2=33
	
		file_num=$(ls |wc -l)
		
		
		nr_files_valid=$(((file_num*num2)/100))
		
		
		
		echo $nr_files_valid
		
		
		echo $file_num
		
		 
		
		
		
		
		newpath=${i:2}
		
		newpath_valid="$path_valid$newpath"
		ls | shuf -n $nr_files_valid | xargs -i mv {} $newpath_valid
		
		
		
		echo "$newpath_valid"
		
		
		
		
		
		echo '####'
		
	

	  cd ..   
	  
	  fi 
	 done	

		
       
		
	
	
	
