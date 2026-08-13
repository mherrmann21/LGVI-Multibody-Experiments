function link = systemDefCantileverBeamHK24(opts)
    %% Define MBS System: Cantilever beam from HK24 simulation study

    arguments
        opts.d      (6,1) double = ones(6,1)*1.2e-4;
        opts.nSeg   (1,1) uint8  = 8;
    end
    link = elara.FlexibleLink;

    link.parentLink = 0;
    link.isCantilever = true;
    link.nSegments = opts.nSeg;
    link.L         = 1;
    link.g_J_B     = eye(4);
    link.Ba = [ eye(3); zeros(3)];
    link.Bc = [ zeros(3); eye(3)];
    link.xiRef = repmat([0;0;0;0;0;1], [1,link.nSegments]);
    link.beamParameters = beamParams_mbsd_stiff_rod;
    link.beamParameters.d = opts.d;
end
