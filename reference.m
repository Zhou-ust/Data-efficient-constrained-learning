
function rt = reference(t)

rt = 0;

if  t <= 10;
    rt = 0;
elseif 10 <=t && t <= 30;
    rt = 15*(t-10);
else 30 <= t ;
    rt = 300;
end
end
