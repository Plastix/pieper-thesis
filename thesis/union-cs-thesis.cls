\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{union-cs-thesis}

%%%% Options
%%%%
%%%% condensed - single spaced, no separate title page, no separate page for
%%%% ToC, etc...
%%%%
%%%% final - double spaced, separate title page, separate page for ToC,
%%%% etc...
%%%%
%%%% honors - double space, separate honors title page, separate page
%%%% for ToC, etc...
%%%%
%%%% default is 'final'


\newif\ifcondensed
\condensedfalse
\DeclareOption{condensed}{\condensedtrue}
\DeclareOption{final}{\condensedfalse}
\newif\ifhonors
\honorsfalse
\DeclareOption{honors}{\honorstrue\condensedfalse}


\ProcessOptions\relax

\RequirePackage{ifthen}
\RequirePackage{xargs}
\RequirePackage{setspace}

\newcommand{\makeabstract}{}
\newcommand{\makepreamble}{}
\newcommand{\dept}[1]{}
\newcommand{\advisor}[1]{}
\newcommand{\authorLF}[1]{}
\newcommand{\bindingOffset}{0in}



\ifcondensed
  \LoadClass[10pt,letterpaper]{article}

  \let\oldtableofcontents\tableofcontents
  \let\oldlistoffigures\listoffigures
  \let\oldlistoftables\listoftables

  \renewcommand{\listoffigures}{}
  \renewcommand{\listoftables}{}

\else
  \LoadClass[10pt,letterpaper]{article}
  \doublespacing

  \let\oldtableofcontents\tableofcontents
  \let\oldlistoffigures\listoffigures
  \let\oldlistoftables\listoftables

  \renewcommand{\tableofcontents}{\clearpage \oldtableofcontents \clearpage}
  \renewcommand{\listoffigures}{\clearpage \oldlistoffigures \clearpage}
  \renewcommand{\listoftables}{\clearpage \oldlistoftables \clearpage}
  
  \ifhonors

    \renewcommand{\dept}[1]{\def \thedept {#1}}
    \renewcommand{\advisor}[1]{\def \theadvisor {#1}}
    \renewcommand{\authorLF}[1]{\def \theauthorLF {#1}}
    
    \RequirePackage{titling}
    \RequirePackage{xstring}
    \RequirePackage{datetime}

    \newdateformat{monthyeardate}{\monthname[\THEMONTH], \THEYEAR}

%    \renewcommand{\bindingOffset}{23pt}

    \posttitle{\par\end{center}\vskip 5em}
    \preauthor{\begin{center}
        \large \lineskip 0.5em%
      By \\
      \vspace{1cm}
      \begin{tabular}[t]{c}}
    \postauthor{\end{tabular}\par\end{center}}
    \predate{\large
      \begin{center} 
        
        \vfill
        
        * * * * * * * * * 
        
        \vfill
        
        Submitted in partial fulfillment\\ 
        of the requirements for \\
        Honors in the \thedept
        
        \vspace{1cm}
        
        UNION COLLEGE \\
        \monthyeardate
    }
    \postdate{
      \pagenumbering{gobble}
      \par\end{center}
      \newpage
    }
    
    \renewcommand{\makeabstract}{
      \setcounter{page}{1}
      \pagenumbering{roman}
      \expandafter\MakeUppercase\expandafter{\theauthorLF} \quad \thetitle. \thedept, \monthyeardate\thedate.
      
      ADVISOR: \theadvisor    
    }


    \fi


\fi

\setlength{\oddsidemargin}{0in}         % Actual left margin - 1 inch. 
\setlength{\evensidemargin}{0in}        % Ditto.                       
\setlength{\textwidth}{6.5in}           % Line length.                 

\usepackage{palatino}
