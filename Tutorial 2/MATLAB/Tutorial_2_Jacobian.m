% File name Jacobian.m
% This function is to calculate the Jacobian matrix, and will be called
%  in the script Tutorial_1_question_2
function J=Tutorial_2_Jacobian(qs,qe,qw)
l_s=0.3;
l_e=0.3;
l_w=0.15;
J=[-l_s*sin(qs) -l_e*sin(qs+qe) -l_w*sin(qs+qe+qw); l_s*cos(qs) l_e*cos(qs+qe) l_w*cos(qs+qe+qw)];
end
% End of function