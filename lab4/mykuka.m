function myrobot = mykuka(DH)
  % Creating Links using our DH table
  for i = 1:size(DH,1)
      L(i) = Link(DH(i,:),'standard')
  end
  % Serializing links
  myrobot =  SerialLink(L, 'name', 'MYKUKA', 'plotopt', {'notiles'})
end


    