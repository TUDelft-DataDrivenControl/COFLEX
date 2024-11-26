function tp=tensorprod3(t1,t2,t3)
    tp=kron(t3,kron(t2,t1));
end