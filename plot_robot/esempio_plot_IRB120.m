% esempio_plot_IRB120.m
% esempio plot robot ABB IRB120

% inputs ACADi frame wrt world 0

ACAD1 = [
    1.0000         0         0         0
         0    1.0000         0         0
         0         0    1.0000    0.2900
         0         0         0    1.0000] ;
           
ACAD2 = [
    1.0000         0    0.0000    0.0000
    0.0000    1.0000   -0.0000   -0.0000
   -0.0000    0.0000    1.0000    0.5600
         0         0         0    1.0000] ;
     
ACAD3 = [
    1.0000         0    0.0000    0.0000
    0.0000    1.0000   -0.0000   -0.0000
   -0.0000    0.0000    1.0000    0.5600
         0         0         0    1.0000] ;

ACAD4 = [
    1.0000   -0.0000    0.0000    0.3020
    0.0000    1.0000   -0.0000   -0.0000
   -0.0000    0.0000    1.0000    0.6300
         0         0         0    1.0000] ;

ACAD5 = [
    1.0000         0    0.0000    0.3020
         0    1.0000   -0.0000   -0.0000
   -0.0000    0.0000    1.0000    0.6300
         0         0         0    1.0000] ;

ACAD6 = [
    1.0000   -0.0000    0.0000    0.3740
    0.0000    1.0000   -0.0000    0.0000
   -0.0000    0.0000    1.0000    0.6300
         0         0         0    1.0000] ;
     
ACADTOOL0o = [
    1.0000   -0.0000    0.0000    0.3740
    0.0000    1.0000   -0.0000    0.0000
   -0.0000    0.0000    1.0000    0.6300
         0         0         0    1.0000] ;

%% Run function
     
plot_IRB120(ACAD1,ACAD2,ACAD3,ACAD4,ACAD5,ACAD6)
plot_IRB120tool(ACAD1,ACAD2,ACAD3,ACAD4,ACAD5,ACAD6,ACADTOOL0o)
delete *.asv % Cleaning