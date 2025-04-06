param (
    [string]$sitePath = "./_site",
    [string]$repoUrl = "https://github.com/bepu/bepuphysics2/blob/master"
)

# Get all HTML files in the site
$htmlFiles = Get-ChildItem -Path $sitePath -Filter "*.html" -Recurse

$linkCount = 0
foreach ($file in $htmlFiles) {
    $content = Get-Content -Path $file.FullName -Raw
    
    # Find links that start with "../" and transform them to GitHub URLs
    $pattern = 'href=["\''](\.\./[^"\'']*)["\'']'
    
    # Use a scriptblock for the replacement to remove the "../" prefix
    $newContent = $content -replace $pattern, {
        $match = $args[0]
        $originalLink = $args[0].Groups[1].Value
        # Remove the "../" prefix for the GitHub URL
        $relativePath = $originalLink -replace '^\.\.\/', ''
        "href=`"$repoUrl/$relativePath`""
    }
    
    # Only write the file if changes were made
    if ($newContent -ne $content) {
        $matches = [regex]::Matches($content, $pattern)
        $linkCount += $matches.Count
        Set-Content -Path $file.FullName -Value $newContent
        Write-Host "Fixed $($matches.Count) links in $($file.Name)"
    }
}

Write-Host "Link transformation complete. Fixed $linkCount links in total."