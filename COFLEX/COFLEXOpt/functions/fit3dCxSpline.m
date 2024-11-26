function [coeff,betaKnots,windKnots,omegaKnots,J]=fit3dCxSpline(beta,wind,omega,data)

    %Hardcode degree to 3
    d=3;
    [Nomega,omegaKnots]=BSplineBasis(omega,d);
    [Nwind,windKnots]=BSplineBasis(wind,d);
    [Nbeta,betaKnots]=BSplineBasis(beta,d);
    
    Jomega = evalBspBasis(Nomega{d+1},d,omegaKnots,omega);
    Jwind  = evalBspBasis(Nwind{d+1},d,windKnots,wind);
    Jbeta  = evalBspBasis(Nbeta{d+1},d,betaKnots,beta);

    J=tensorprod3(Jomega,Jwind,Jbeta);

    %Remove numerically small elements
    J=J.*(J>1e6*eps);
    fV=reshape(data,1,[]);
    
    setup = struct('type','ilutp','droptol',1e-10);
    [L,U] = ilu(J,setup);
    coeff = lsqr(J,fV',1e-10,10000,L,U);  
end