all: SLAM_Report.pdf

*.tex: 

%.pdf: %.tex %.bib
	pdflatex SLAM_Report.tex
	bibtex SLAM_Report.aux
	pdflatex SLAM_Report.tex
	pdflatex SLAM_Report.tex

clean:
	rm *.log *.aux *.bbl *.bld *.dvi
