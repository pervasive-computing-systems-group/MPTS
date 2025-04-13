#!/bin/bash

set -e

echo "Running Gradient Search on Case Study"
../build/find-assignment "CaseStudy/case_study.txt" 6 1 CaseStudy/ >> CaseStudy/output.txt&
wait

echo "Running Matching + Gradient Search on Case Study"
../build/find-assignment "CaseStudy/case_study.txt" 7 1 CaseStudy/ >> CaseStudy/output.txt&
wait

echo "Running Complete on Case Study"
../build/find-assignment "CaseStudy/case_study.txt" 1 1 CaseStudy/ >> CaseStudy/output.txt&
wait

