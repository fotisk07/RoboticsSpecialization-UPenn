function testSim()
  format compact
  set(groot,'defaultLineLineWidth',1)


  close('all')
  figure
  hold all
  simode(1, 50);
  simode(5, 50);
  simode(10, 50);
  hold off
end