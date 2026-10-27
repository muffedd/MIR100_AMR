function [declaredMass, hasInertial, linkNames] = urdf_link_masses(urdfPath)
%URDF_LINK_MASSES Parse per-link inertial mass declarations from a URDF file.
%
%   [declaredMass, hasInertial, linkNames] = URDF_LINK_MASSES(urdfPath)
%   reads the URDF as text and returns, for every <link name="...">:
%     declaredMass - containers.Map name -> declared <inertial><mass value>
%                    in kg (NaN when the link has no inertial mass)
%     hasInertial  - containers.Map name -> logical, true when the link
%                    contains an <inertial> element
%     linkNames    - cellstr of link names in document order
%
%   This is SOURCE evidence: what the URDF declares, independent of what
%   importrobot does with it. MATLAB importrobot behavior for links
%   WITHOUT <inertial> is UNVERIFIED (no MATLAB in this environment), so
%   load_mir100 normalizes those bodies to Mass = 0 explicitly.

    txt = fileread(urdfPath);
    txt = regexprep(txt, '(?s)<!--.*?-->', '');
    % MATLAB regexp uses \b for backspace, not a word boundary.
    [~, tagEnd, tags] = regexp(txt, '<link\s[^>]*>', 'start', 'end', 'match');
    declaredMass = containers.Map();
    hasInertial = containers.Map();
    linkNames = cell(1, numel(tags));
    for i = 1:numel(tags)
        nameToken = regexp(tags{i}, 'name="([^"]+)"', 'tokens', 'once');
        name = nameToken{1};
        content = '';
        if isempty(regexp(tags{i}, '/>\s*$', 'once'))
            tail = txt(tagEnd(i)+1:end);
            closeAt = strfind(tail, '</link>');
            if isempty(closeAt), error('urdf_link_masses:invalidXML','Unclosed link %s.',name); end
            content = tail(1:closeAt(1)-1);
        end
        linkNames{i} = name;
        hasInertial(name) = ~isempty(strfind(content, '<inertial'));
        m = regexp(content, '<mass\s+[^>]*?value="([0-9.eE+\-]+)"', 'tokens', 'once');
        if ~isempty(m)
            declaredMass(name) = str2double(m{1});
        else
            declaredMass(name) = NaN;
        end
    end
    if isempty(linkNames)
        error('urdf_link_masses:noLinks','No links parsed from %s.',urdfPath);
    end
end
