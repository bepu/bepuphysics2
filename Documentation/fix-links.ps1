param (
    [string]$sitePath = "./_site",
    [string]$repoUrl = "https://github.com/bepu/bepuphysics2/blob/master"
)

# Get all HTML files in the site
$htmlFiles = Get-ChildItem -Path $sitePath -Filter "*.html" -Recurse

$linkCount = 0
foreach ($file in $htmlFiles) {
    $content = Get-Content -Path $file.FullName -Raw
    
    # Find links like "../Folder/File.cs" and transform them to GitHub URLs
    $pattern = 'href=["\''](\.\./[^"\'']*(\.cs|\.csproj))(#L\d+)?["\'']'
    
    $newContent = $content -replace $pattern, "href=`"$repoUrl/`$1`$3`""
    
    # Only write the file if changes were made
    if ($newContent -ne $content) {
        $matches = [regex]::Matches($content, $pattern)
        $linkCount += $matches.Count
        Set-Content -Path $file.FullName -Value $newContent
        Write-Host "Fixed $($matches.Count) links in $($file.Name)"
    }
}

Write-Host "Link transformation complete. Fixed $linkCount links in total."