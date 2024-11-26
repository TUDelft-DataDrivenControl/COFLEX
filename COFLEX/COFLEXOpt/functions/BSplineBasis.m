function [N,knots]=BSplineBasis(x,d)
   
   % to create basis functions (N) for B splines
   nm  = d+1;

   %Not-a-knot requires throwing out the knots which are almost at the
   %edges. For d=3 it is the second and second to last knots which are not
   %knots

   nSkip = (nm-2)/2+1;
   knots = [x(1)*ones(1,nm)  x(nSkip+1:end-nSkip) x(end)*ones(1,nm)]; % For a clamped B-spline, increase the multiplicity of first and last knots

   %N=zeros(length(knots),p); %Degree 0
   %N{d}=

   m     = length(knots)-1; % according to convention, u(1) is u_0 and u(len(knots)) is u_m
   Ndiag = diff(knots)>0;   % logical vector containing 1 where subsequent knots are different in the knots vector
   N{1}  = mat2cell(diag(Ndiag),ones(1,m),ones(1,m));

   %N{1}(1:d)=zeros(d,1);
   %N{1}(end-d+1:end)=zeros(d,1);

   for p=1:d

     nb=m-p;
     N{p+1}=cell(m,nb);

     for basis=1:m-p
         for intval=basis:(p+basis)
            N{p+1}{intval,basis}=cN(knots,p,intval,basis,N{p});
         end      
     end

   end

end

function cx=cN(u,p,intval,basis,N) %u - knots, p degree, i index, j basis
    Nb=N{intval,basis};

    if isempty(Nb)
       Nb=0;
    end

    Nb1=N{intval,basis+1};
    if isempty(Nb1)
       Nb1=0;
    end
    
    den=u(basis+p)-u(basis);
    if den==0
        cix=0;
        ci0=0;
    else
        cix=1/den*Nb; %*x*N_basis,p-1
        ci0=-u(basis)/den*Nb; %*N_basis,p-1
    end

    den=u(basis+p+1)-u(basis+1);
    if den==0
        cipx=0;
        cip0=0;
    else
        cipx=-1/den*Nb1; %*x*N_basis+1,p-1
        cip0=u(basis+p+1)/den*Nb1; %*N_basis+1,p-1
    end

    cx=zeros(1,p+1);
    cc=[0 cix+cipx]+[ci0+cip0 0];
    cx(1:length(cc))=cc;
   
end