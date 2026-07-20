#!/bin/bash
rm -rf doc
mkdir  doc
protoc \
  --plugin=protoc-gen-doc=$HOME/protoc-gen-doc \
  --doc_out=./doc \
  --doc_opt=html,index.html \
  *.proto
