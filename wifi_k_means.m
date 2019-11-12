function wifi_k_means(varargin)
    % parse options and put in 'params'
  defaults = struct('A',1, 'B',magic(3));  %define default values

  params = struct(varargin{:});
  for f = fieldnames(defaults)',
    if ~isfield(params, f{1}),
      params.(f{1}) = defaults.(f{1});
    end
  end
  
  % Main code starts here

end