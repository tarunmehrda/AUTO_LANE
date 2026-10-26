"""
compile_report_and_slides.py
----------------------------
Compiles the Markdown thesis report and slide deck into polished,
publication-quality HTML documents with academic styling, embedded
figures, and print-ready CSS for university submission.
"""

import os
import re
import markdown

REPORT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(REPORT_DIR)

def compile_thesis_html():
    md_path = os.path.join(REPORT_DIR, "Major_Project_Report_Final.md")
    html_path = os.path.join(REPORT_DIR, "Major_Project_Report_Final.html")
    
    with open(md_path, "r", encoding="utf-8") as f:
        md_text = f.read()

    # Convert Markdown to HTML with tables, code fences, and toc
    html_body = markdown.markdown(
        md_text,
        extensions=["tables", "fenced_code", "toc", "attr_list"]
    )

    # Academic stylesheet for printing and onscreen viewing
    css = """
    <style>
        @page {
            size: A4;
            margin: 25mm 20mm 25mm 25mm;
            @bottom-right {
                content: counter(page);
            }
        }
        body {
            font-family: 'Times New Roman', Times, serif;
            font-size: 12pt;
            line-height: 1.6;
            color: #111;
            background-color: #f8f9fa;
            margin: 0;
            padding: 20px;
        }
        .container {
            max-width: 900px;
            margin: 0 auto;
            background: #ffffff;
            padding: 50px 60px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.1);
            border-radius: 4px;
        }
        h1, h2, h3, h4 {
            font-family: 'Arial', sans-serif;
            color: #1a365d;
            font-weight: 700;
        }
        h1 {
            font-size: 20pt;
            text-align: center;
            border-bottom: 2px solid #1a365d;
            padding-bottom: 8px;
            margin-top: 30px;
        }
        h2 {
            font-size: 15pt;
            border-bottom: 1px solid #cbd5e0;
            padding-bottom: 5px;
            margin-top: 25px;
        }
        h3 {
            font-size: 13pt;
            margin-top: 20px;
        }
        table {
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
            font-size: 10.5pt;
        }
        th, td {
            border: 1px solid #cbd5e0;
            padding: 8px 12px;
            text-align: left;
        }
        th {
            background-color: #2b6cb0;
            color: #ffffff;
            font-family: 'Arial', sans-serif;
            font-weight: bold;
        }
        tr:nth-child(even) {
            background-color: #f7fafc;
        }
        pre {
            background: #edf2f7;
            padding: 12px 16px;
            border-radius: 4px;
            overflow-x: auto;
            font-family: 'Courier New', monospace;
            font-size: 9.5pt;
            border-left: 4px solid #3182ce;
        }
        code {
            background: #edf2f7;
            padding: 2px 5px;
            border-radius: 3px;
            font-family: 'Courier New', monospace;
            font-size: 10pt;
        }
        hr {
            border: none;
            border-top: 1px dashed #a0aec0;
            margin: 30px 0;
        }
        .figure-box {
            text-align: center;
            margin: 25px 0;
        }
        .figure-box img {
            max-width: 90%;
            height: auto;
            border: 1px solid #e2e8f0;
            border-radius: 4px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.08);
        }
        .caption {
            font-size: 10pt;
            font-style: italic;
            color: #4a5568;
            margin-top: 6px;
        }
        @media print {
            body {
                background: none;
                padding: 0;
            }
            .container {
                box-shadow: none;
                padding: 0;
                max-width: 100%;
            }
            .no-print {
                display: none;
            }
            h1 {
                page-break-before: always;
            }
            h1:first-of-type {
                page-break-before: avoid;
            }
        }
        .print-btn {
            position: fixed;
            top: 20px;
            right: 20px;
            background: #2b6cb0;
            color: white;
            padding: 10px 18px;
            font-family: Arial, sans-serif;
            font-weight: bold;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            box-shadow: 0 2px 10px rgba(0,0,0,0.2);
            z-index: 1000;
        }
        .print-btn:hover {
            background: #2c5282;
        }
    </style>
    """

    full_html = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Major Project Report — FPGA-Accelerated Lane Detection</title>
    {css}
</head>
<body>
    <button class="print-btn no-print" onclick="window.print()">🖨️ Print to PDF / Paper</button>
    <div class="container">
        {html_body}
    </div>
</body>
</html>
"""

    with open(html_path, "w", encoding="utf-8") as f:
        f.write(full_html)
        
    print(f"[1/2] Compiled Thesis Report: {html_path}")


def compile_slides_html():
    md_path = os.path.join(REPORT_DIR, "Presentation_Slide_Deck.md")
    html_path = os.path.join(REPORT_DIR, "Presentation_Slide_Deck.html")

    with open(md_path, "r", encoding="utf-8") as f:
        md_text = f.read()

    # Split markdown slides by '## SLIDE'
    slides_raw = re.split(r'(?=## SLIDE )', md_text)
    header_content = slides_raw[0] if len(slides_raw) > 1 else ""
    slide_sections = slides_raw[1:] if len(slides_raw) > 1 else slides_raw

    slides_html_list = []
    for idx, slide_md in enumerate(slide_sections):
        slide_body = markdown.markdown(slide_md, extensions=["tables", "fenced_code"])
        slides_html_list.append(f"""
        <div class="slide" id="slide-{idx+1}">
            <div class="slide-header">
                <span>UIET, Panjab University — Major Project Defense</span>
                <span>Slide {idx+1} of {len(slide_sections)}</span>
            </div>
            <div class="slide-content">
                {slide_body}
            </div>
        </div>
        """)

    all_slides_html = "\n".join(slides_html_list)

    slide_css = """
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #0f172a;
            color: #f8fafc;
            margin: 0;
            padding: 0;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        .slide-nav {
            position: sticky;
            top: 0;
            width: 100%;
            background: #1e293b;
            padding: 12px 24px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            box-shadow: 0 4px 10px rgba(0,0,0,0.3);
            z-index: 100;
        }
        .nav-title {
            font-size: 14pt;
            font-weight: bold;
            color: #38bdf8;
        }
        .nav-controls button {
            background: #38bdf8;
            color: #0f172a;
            border: none;
            font-weight: bold;
            padding: 8px 16px;
            margin-left: 8px;
            border-radius: 4px;
            cursor: pointer;
        }
        .nav-controls button:hover {
            background: #7dd3fc;
        }
        .slide {
            width: 90%;
            max-width: 1100px;
            min-height: 650px;
            background: #1e293b;
            margin: 30px 0;
            border-radius: 12px;
            box-shadow: 0 10px 25px rgba(0,0,0,0.5);
            border: 1px solid #334155;
            padding: 40px;
            box-sizing: border-box;
            display: flex;
            flex-direction: column;
        }
        .slide-header {
            display: flex;
            justify-content: space-between;
            font-size: 10pt;
            color: #94a3b8;
            border-bottom: 1px solid #334155;
            padding-bottom: 10px;
            margin-bottom: 20px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .slide-content h2 {
            color: #38bdf8;
            font-size: 20pt;
            margin-top: 0;
        }
        .slide-content h3 {
            color: #f59e0b;
            font-size: 14pt;
            margin-top: 20px;
        }
        .slide-content ul {
            font-size: 12pt;
            line-height: 1.7;
        }
        .slide-content table {
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
        }
        .slide-content th, .slide-content td {
            border: 1px solid #475569;
            padding: 8px 12px;
            text-align: left;
        }
        .slide-content th {
            background: #0ea5e9;
            color: #0f172a;
        }
        .slide-content pre {
            background: #0f172a;
            padding: 15px;
            border-radius: 6px;
            border-left: 4px solid #38bdf8;
            font-size: 10pt;
            color: #e2e8f0;
            overflow-x: auto;
        }
        blockquote {
            background: rgba(56, 189, 248, 0.1);
            border-left: 4px solid #38bdf8;
            margin: 15px 0;
            padding: 12px 18px;
            font-style: italic;
            color: #bae6fd;
            border-radius: 0 4px 4px 0;
        }
    </style>
    """

    full_slides_html = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Presentation Slide Deck — UIET Panjab University</title>
    {slide_css}
</head>
<body>
    <div class="slide-nav">
        <span class="nav-title">FPGA-Accelerated Lane Detection — B.E. Capstone Defense</span>
        <div class="nav-controls">
            <button onclick="window.print()">🖨️ Print Slides</button>
        </div>
    </div>
    {all_slides_html}
</body>
</html>
"""

    with open(html_path, "w", encoding="utf-8") as f:
        f.write(full_slides_html)

    print(f"[2/2] Compiled Presentation Deck: {html_path}")


if __name__ == "__main__":
    compile_thesis_html()
    compile_slides_html()
