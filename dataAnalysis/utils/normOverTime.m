function n = normOverTime(e)
    n = zeros(length(e),1);
    
    for i = 1:length(e)
        n(i) = norm(e(i));
    end 
end



% Norm of error computation
% e_vcn_vio_norm = zeros(time_samples,1);
% 
% for ii = 1:time_samples
%     e_vcn_vio_norm(ii) = norm(e_vcn_vio(ii));
% end