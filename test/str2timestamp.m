function timestamp = str2timestamp(str)
    string = str{1,1};
    C1 = strsplit(string);
    
    C2 = strsplit(C1{1,2}, ':');
    
    h = C2(1, 1);
    m = C2(1, 2);
    s = C2(1, 3);
    H = str2double(h{1});
    M = str2double(m{1});
    S = str2double(s{1});
    timestamp = H*60*60*1000 + M*60*1000 + S*1000;
    
    %disp(timestamp);
end
