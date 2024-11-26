 function spline=createParCasadiSpline(name,knots,coeff)
    import casadi.*
    %Create a parametric BSpline for 3D                                   
    dims = length(knots);
    hardcoded=~isa(coeff,'MX');
    
    inp         = MX.sym('inp',dims);
    if(hardcoded)
        coeffSym=coeff;
        x={inp};
    else
        coeffSym = MX.sym([name '_Coeff'],length(coeff)); 
        x={inp,coeffSym};
    end
    splineMx   = bspline(inp, coeffSym, knots , 3*ones(1,dims), 1, struct());
    splineF    = casadi.Function([name '_spline'],x,{splineMx},struct());

    if(hardcoded)
       spline  = @(v) splineF(v);
    else
       spline  = @(v) splineF(v,coeff);
    end
 end