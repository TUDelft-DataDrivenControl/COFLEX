function evalSpline=evalBspBasis(basis,d,knots,v)
    nVals=length(v);
    evalSpline=spalloc(nVals,size(basis,2),nVals*(d+1)+d*2*2);
    for r=1:nVals
        vv=v(r);
        intVals=(vv>knots(1:end-1) & vv<knots(2:end)) | vv==knots(1:end-1);
        %Hack for closed group of last knot
        if(vv==knots(end))
            intVals(end-d)=1;
        end
        %intVals=(vv>knots & vv<knots) | vv==knots;
        polys=basis(intVals,:);
        vvp=[];
        for k=0:d
            vvp(end+1)=vv^k;
        end
        for k=1:size(polys,2)
            pp=zeros(1,d+1);
            for m=1:size(polys,1)
                if ~isempty(polys{m,k})
                    p2=polys{m,k};
                    pp=pp+p2;
                end
            end
            evalSpline(r,k)=pp*vvp';
        end
    end
end