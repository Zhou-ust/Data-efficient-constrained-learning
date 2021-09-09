function Xi = get_OriginalData(X,Originalstate)

if size(X,1) ==3
    De = 2;
elseif size(X,1) ==1
    De = 1;
end

Xi = zeros(size(X(1:De,:)));
for j = 1:size(Xi,2)
    if j == 1
        Xi(1:De,1) =  X(1:De,1) + Originalstate;
    else
        Xi(1:De,j) =  Xi(1:De,j-1) + X(1:De,j);
    end
end

end
