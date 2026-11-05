' Converts every STEP file in "version 1\step" and "version 2\step" to native
' SOLIDWORKS files, saved in a new "sldprt" folder inside each version folder.
' Not yet run in SOLIDWORKS: see SOLIDWORKS.md, and check the first results.

Option Explicit

Const swSaveAsCurrentVersion As Long = 0
Const swSaveAsOptions_Silent As Long = 1
Const swDocPART As Long = 1
Const swDocASSEMBLY As Long = 2

Sub main()
    Dim swApp As Object
    Set swApp = Application.SldWorks

    Dim root As String
    root = InputBox("Folder that contains 'version 1' and 'version 2':", "STEP to SOLIDWORKS", "D:\3d print")
    If root = "" Then Exit Sub

    Dim report As String
    ConvertFolder swApp, root & "\version 1", report
    ConvertFolder swApp, root & "\version 2", report
    MsgBox report, vbInformation, "STEP to SOLIDWORKS"
End Sub

Private Sub ConvertFolder(swApp As Object, versionDir As String, report As String)
    Dim stepDir As String, outDir As String
    stepDir = versionDir & "\step\"
    outDir = versionDir & "\sldprt\"
    If Dir(stepDir, vbDirectory) = "" Then
        report = report & "Folder not found: " & stepDir & vbCrLf
        Exit Sub
    End If
    If Dir(outDir, vbDirectory) = "" Then MkDir outDir

    ' Collect names first: opening files in SOLIDWORKS can disturb a running Dir loop.
    Dim names As New Collection, f As String
    f = Dir(stepDir & "*.step")
    Do While f <> ""
        names.Add f
        f = Dir()
    Loop

    Dim item As Variant, good As Long, bad As Long
    For Each item In names
        If ConvertOne(swApp, stepDir & item, outDir, CStr(item)) Then
            good = good + 1
        Else
            bad = bad + 1
            report = report & "FAILED: " & stepDir & item & vbCrLf
        End If
    Next item
    report = report & versionDir & ": " & good & " saved, " & bad & " failed" & vbCrLf
End Sub

Private Function ConvertOne(swApp As Object, stepPath As String, outDir As String, fileName As String) As Boolean
    Dim importData As Object, doc As Object, errs As Long, warns As Long
    Set importData = swApp.GetImportFileData(stepPath)
    Set doc = swApp.LoadFile4(stepPath, "r", importData, errs)
    If doc Is Nothing Then Exit Function

    Dim ext As String
    Select Case doc.GetType
        Case swDocPART: ext = ".SLDPRT"
        Case swDocASSEMBLY: ext = ".SLDASM"
        Case Else
            swApp.CloseDoc doc.GetTitle
            Exit Function
    End Select

    Dim target As String
    target = outDir & Left(fileName, Len(fileName) - Len(".step")) & ext
    ConvertOne = doc.Extension.SaveAs(target, swSaveAsCurrentVersion, swSaveAsOptions_Silent, Nothing, errs, warns)
    swApp.CloseDoc doc.GetTitle
End Function
