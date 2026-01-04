function myrobot = mykuka_search(delta)
  % Initialize DH table with perturbations
  DH = [[0, 400, 25, pi/2]; 
        [0, 0, 315, 0]; 
        [0, 0, 35, pi/2]; 
        [0, 365, 0, -pi/2]; 
        [0, 0, 0, pi/2]; 
        [0, 161.44+delta(2), -296.33+delta(1), 0]];
  % Creating Links using our DH table
  for i = 1:size(DH,1)
      L(i) = Link(DH(i,:),'standard');
  end
  % Serializing links
  myrobot =  SerialLink(L, 'name', 'MYKUKA');
end


    