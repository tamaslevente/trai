clc;
close all;


 point_count = 1024;
 density_distribution = {'uniform',1,0.1};
     
% Define a starting folder.
start_path = fullfile('/home/alex-pop/Desktop/Doctorat/Backups/Trial_Test_Valid_mat/Train_Test/test');

Ceva=[]

% Ask user to confirm or change.
topLevelFolder = uigetdir(start_path)


d = dir(topLevelFolder)

a=size(d);

length_d=a(1)

original_path=topLevelFolder

original_path=strcat(original_path,'/')

 %for i=3:length_d
 for i=4:4
%     
 class_path=d(i).name
 
 complete_class_path=strcat(original_path,class_path)
 
 d_class=dir(complete_class_path)
 
 a_class=size(d_class);

length_d_class=a_class(1);
 
 class_path_new=strcat(complete_class_path,'/');
 
 %for j=3:length_d_class
 for j=4:4
     
    model_path=d_class(j).name

   complete_path=strcat(class_path_new,model_path)

   complete_path_model=strcat(complete_path,'/model.off')


    Test_pointcloud= pcloud_from_mesh(point_count,density_distribution,complete_path_model)
     complete_path_mat=strcat(complete_path,'/model.mat')
 
     points=Test_pointcloud.points;
     
     if (j<100)
     Ceva=[Ceva points];
     end

     save(complete_path_mat,'points');
      
end
 
 
 
end


%

% for i=3:length_d
%     
% model_path=d(i).name
% 
% complete_path=strcat(original_path,model_path)
% 
% complete_path_model=strcat(complete_path,'/model.off')
% 
% 
% Test_pointcloud= pcloud_from_mesh(point_count,density_distribution,complete_path_model)
% complete_path_mat=strcat(complete_path,'/model.mat')
% 
% points=Test_pointcloud.points;
% 
% Ceva=[Ceva points];
% 
% 
% save(complete_path_mat,'points');
% end
% %%



%save('/home/alex-pop/Downloads/training_set/TEst_program/1a6ad7a24bb89733f412783097373bdc/model.mat','Test_pointcloud')
    
%%

for i=1:3:3

    [i,i+1,i+2]
    
    figure(i);
    plot3(Ceva(:,i),Ceva(:,i+1),Ceva(:,i+2),"*")
end



