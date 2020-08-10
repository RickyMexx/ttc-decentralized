function D = data_store(D, idx, q, dq, ddq, e, u)
%DATA_STORE Stores in data matrix D at the index i the current dynamic
%system variables.
%
% D    : Data matrix [5xn]
% idx  : current time index
% q,dq,ddq,e,u : system variables
%D(:, idx) = [q;dq;ddq;e;u];
D(:, idx) = [e;q;dq;ddq;u];
end

