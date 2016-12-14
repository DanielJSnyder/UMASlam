all: SLAM_Report.pdf

*.tex: 

%.pdf: %.tex %.bib
	pdflatex $^

clean:
	rm *.log *.aux *.bbl 
