import zipfile
from pathlib import Path


def escape(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


def md_to_docx(md_path: Path, docx_path: Path) -> None:
    lines = md_path.read_text(encoding="utf-8").splitlines()
    paragraphs = []
    for line in lines:
        if line.strip() == "":
            paragraphs.append("<w:p/>")
        else:
            paragraphs.append(
                "<w:p><w:r><w:t xml:space=\"preserve\">{}</w:t></w:r></w:p>".format(
                    escape(line)
                )
            )

    content_types = """<?xml version='1.0' encoding='UTF-8' standalone='yes'?>
<Types xmlns='http://schemas.openxmlformats.org/package/2006/content-types'>
  <Default Extension='rels' ContentType='application/vnd.openxmlformats-package.relationships+xml'/>
  <Default Extension='xml' ContentType='application/xml'/>
  <Override PartName='/word/document.xml' ContentType='application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml'/>
  <Override PartName='/word/styles.xml' ContentType='application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml'/>
</Types>
""".strip()

    rels = """<?xml version='1.0' encoding='UTF-8' standalone='yes'?>
<Relationships xmlns='http://schemas.openxmlformats.org/package/2006/relationships'>
  <Relationship Id='rId1' Type='http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument' Target='word/document.xml'/>
</Relationships>
""".strip()

    doc_rels = """<?xml version='1.0' encoding='UTF-8' standalone='yes'?>
<Relationships xmlns='http://schemas.openxmlformats.org/package/2006/relationships'>
  <Relationship Id='rId1' Type='http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles' Target='styles.xml'/>
</Relationships>
""".strip()

    styles = """<?xml version='1.0' encoding='UTF-8' standalone='yes'?>
<w:styles xmlns:w='http://schemas.openxmlformats.org/wordprocessingml/2006/main'>
  <w:style w:type='paragraph' w:default='1' w:styleId='Normal'>
    <w:name w:val='Normal'/>
    <w:qFormat/>
    <w:pPr/>
    <w:rPr/>
  </w:style>
</w:styles>
""".strip()

    body = "\n    ".join(paragraphs)
    document_xml = """<?xml version='1.0' encoding='UTF-8' standalone='yes'?>
<w:document xmlns:w='http://schemas.openxmlformats.org/wordprocessingml/2006/main'>
  <w:body>
    {}
    <w:sectPr>
      <w:pgSz w:w='12240' w:h='15840'/>
      <w:pgMar w:top='1440' w:right='1440' w:bottom='1440' w:left='1440'/>
    </w:sectPr>
  </w:body>
</w:document>
""".strip().format(body)

    with zipfile.ZipFile(docx_path, "w") as zf:
        zf.writestr("[Content_Types].xml", content_types)
        zf.writestr("_rels/.rels", rels)
        zf.writestr("word/_rels/document.xml.rels", doc_rels)
        zf.writestr("word/styles.xml", styles)
        zf.writestr("word/document.xml", document_xml)


if __name__ == "__main__":
    md_file = Path(__file__).with_name("Greenhouse_Weed_Trial_Plan.md")
    docx_file = Path(__file__).with_name("Greenhouse_Weed_Trial_Plan.docx")
    md_to_docx(md_file, docx_file)
    print(f"Generated {docx_file}")


