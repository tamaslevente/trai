clear all;
close all;
clc;

X= readmatrix('All_data.csv');

nr_pozitii=3;

threshold_value_1=0.1
position_1=1

coverage_difference_predictions=X(:,1);
coverage_error_nochange=X(:,2);
coverage_error_permut=X(:,3);

coverage_nochange=X(:,4);
coverage_permut=X(:,5);
coverage_greedy=X(:,6);

coverage_difference_predictions_threshold=coverage_difference_predictions;

figure('Name',"Coverage difference predictions")
histogram(coverage_difference_predictions,5)
figure('Name',"Coverage error nochange")
histogram(coverage_error_nochange,5)
figure('Name',"Coverage error permut")
histogram(coverage_error_permut,5)
%%

% 
% for i=1:length(X)
%     if (abs(coverage_difference_predictions_threshold(i))<threshold_value_1)
%        coverage_difference_predictions_threshold(i)=0;
%     end
% end
% 
% 
% plot(coverage_difference_predictions_threshold)


fileID = fopen('Problem_views.txt','w');


disp('Top 3 positions differences in prediction permut bad')
[sorted_coverage_difference , positions_coverage_difference] = sort(coverage_difference_predictions,'descend');
positions_coverage_difference_top3=positions_coverage_difference(1:nr_pozitii)

for i=1:nr_pozitii
   A= [positions_coverage_difference(i)-1,coverage_greedy(positions_coverage_difference_top3(i)),coverage_nochange(positions_coverage_difference_top3(i)),coverage_permut(positions_coverage_difference_top3(i))];
   [coverage_greedy(positions_coverage_difference_top3(i)),coverage_nochange(positions_coverage_difference_top3(i)),coverage_permut(positions_coverage_difference_top3(i))]
   fprintf(fileID,'%d %6.3f %6.3f %6.3f\n',A);
end

disp('Top 3 positions differences in prediction nochange bad')
[sorted_coverage_difference_ascend , positions_coverage_difference_ascend] = sort(coverage_difference_predictions,'ascend');
positions_coverage_difference_ascend_top3=positions_coverage_difference_ascend(1:nr_pozitii)

for i=1:nr_pozitii
    A=[positions_coverage_difference_ascend(i)-1,coverage_greedy(positions_coverage_difference_ascend_top3(i)),coverage_nochange(positions_coverage_difference_ascend_top3(i)),coverage_permut(positions_coverage_difference_ascend_top3(i))];
    [coverage_greedy(positions_coverage_difference_ascend_top3(i)),coverage_nochange(positions_coverage_difference_ascend_top3(i)),coverage_permut(positions_coverage_difference_ascend_top3(i))]
    fprintf(fileID,'%d %6.3f %6.3f %6.3f\n',A);
end

disp('Top 3 positions bad coverage nochange')
[sorted_coverage_error_nochange , positions_coverage_error_nochange] = sort(coverage_error_nochange,'descend');
positions_coverage_error_nochange_top3=positions_coverage_error_nochange(1:nr_pozitii)

for i=1:nr_pozitii
    A=[positions_coverage_error_nochange(i)-1,coverage_greedy(positions_coverage_error_nochange_top3(i)),coverage_nochange(positions_coverage_error_nochange_top3(i)),coverage_permut(positions_coverage_error_nochange_top3(i))];
    [coverage_greedy(positions_coverage_error_nochange_top3(i)),coverage_nochange(positions_coverage_error_nochange_top3(i)),coverage_permut(positions_coverage_error_nochange_top3(i))]
    fprintf(fileID,'%d %6.3f %6.3f %6.3f\n',A);
end

disp('Top 3 positions bad coverage permut')
[sorted_coverage_error_permut , positions_coverage_error_permut] = sort(coverage_error_permut,'descend');
positions_coverage_error_permut_top3=positions_coverage_error_permut(1:nr_pozitii)

for i=1:nr_pozitii
    A=[positions_coverage_error_permut(i)-1,coverage_greedy(positions_coverage_error_permut_top3(i)),coverage_nochange(positions_coverage_error_permut_top3(i)),coverage_permut(positions_coverage_error_permut_top3(i))];
    [coverage_greedy(positions_coverage_error_permut_top3(i)),coverage_nochange(positions_coverage_error_permut_top3(i)),coverage_permut(positions_coverage_error_permut_top3(i))]
    fprintf(fileID,'%d %6.3f %6.3f %6.3f\n',A);
end




%%


figure('Name',"Coverage difference predictions")
plot(coverage_difference_predictions)

figure('Name',"Coverage error nochange")
plot(coverage_error_nochange,'*')

figure('Name',"Coverage error permut")
plot(coverage_error_permut,'*')

figure('Name',"Coverage Nochange")
plot(coverage_nochange,'*')

figure('Name',"Coverage permut")
plot(coverage_permut,'*')

figure('Name',"Coverage Greedy")
plot(coverage_greedy,'*')


%%



