function S = calcS(P00,P01,P10,P11,velObsVar)
%CALCS
%    S = CALCS(P00,P01,P10,P11,VELOBSVAR)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    17-Nov-2019 20:05:15

S = reshape([P00+velObsVar,P10,P01,P11+velObsVar],[2, 2]);