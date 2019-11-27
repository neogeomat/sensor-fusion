function [mac_dec] = getUniqueMac(file)
fid = fopen(file);
hdr = fgetl(fid);
fclose(fid);
macs = regexp(hdr,',','split');
mac_dec = zeros(1,length(macs)-5);
% for ap = 1:length(macs) - 5
for mac_idx = 1:(length(macs) -5)
    MAC_str = macs(mac_idx);
    MAC_dec_array=sscanf(MAC_str{1,1},'%x:%x:%x:%x:%x:%x');
    mac_dec(mac_idx) = MAC_dec_array(1)*256^5 ...
        + MAC_dec_array(2)*256^4 ...
        + MAC_dec_array(3)*256^3 ...
        + MAC_dec_array(4)*256^2 ...
        + MAC_dec_array(5)*256 ...
        + MAC_dec_array(6);
end
end