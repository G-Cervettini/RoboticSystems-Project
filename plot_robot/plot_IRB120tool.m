% plot_IRB120tool.m
% file to plot robot ABB IRB120 with simulink file graph_IRB120.slx

%inputs: matrixes (4x4)of reference frames ACADi (i = 1-6) of CAD frames on each
%robot link, according to image files of links BASE.STL, LINK_1.STL,
%LINK_2.STL, LINK_3.STL, LINK_4.STL, LINK_5.STL, LINK_6.STL, Waterjet.stl

function plot_IRB120(ACAD10o,ACAD20o,ACAD30o,ACAD40o,ACAD50o,ACAD60o,ACADTOOL0o)

    for i = 1:6
        eval(sprintf('[theta%i,axisH%i] = r2angvec_POLITO(ACAD%i0o(1:3,1:3));',i,i,i))
        eval(sprintf('Tr%i = ACAD%i0o(1:3,4);',i,i))
    end
    
    [thetaTOOL,axisHTOOL] = r2angvec_POLITO(ACADTOOL0o(1:3,1:3)) ;
    TrTOOL = ACADTOOL0o(1:3,4) ;
    
    options = simset('SrcWorkspace','current');
    
    sim('graph_IRB120tool',[],options)
    
end