param (
    [string]$sitePath = "./_site",
    [string]$repoUrl = "https://github.com/bepu/bepuphysics2/blob/master"
)

# Get all HTML files in the site, excluding those in the api/ directory
$htmlFiles = Get-ChildItem -Path $sitePath -Filter "*.html" -Recurse | Where-Object { $_.FullName -notmatch "\\api\\" }

$linkCount = 0
foreach ($file in $htmlFiles) {
    $content = Get-Content -Path $file.FullName -Raw
    $originalContent = $content
    
    Write-Host "Considering file: $($file.FullName)"
    
    # Pattern to find all href links
    $pattern = '(href=["''])([^"'']*)(["''])'
    
    # Use a scriptblock for the replacement
    $newContent = [regex]::Replace($content, $pattern, {
        param($match)
        $prefix = $match.Groups[1].Value  # href=" or href='
        $path = $match.Groups[2].Value    # path
        $suffix = $match.Groups[3].Value  # " or '
        
        # Skip if it's already an absolute URL or has special protocols
        if ($path -match '^(https?:|mailto:|#|javascript:)') {
            return $match.Value
        }
        
        # Skip if it's a reference to an HTML file (we don't want to rewrite these)
        if ($path -match '\.html$') {
            return $match.Value
        }
        
        # Case 1: Path starts with "../" (from Documentation directory)
        if ($path -match '^\.\./') {
            $relativePath = $path -replace '^\.\.\/', ''
            Write-Host "  Rewriting '../' link: $path -> $repoUrl/$relativePath"
            return "$prefix$repoUrl/$relativePath$suffix"
        }
        
        # Case 2: Path ends with ".cs" (code file reference)
        if ($path -match '\.cs$') {
            Write-Host "  Rewriting '.cs' link: $path -> $repoUrl/$path"
            return "$prefix$repoUrl/$path$suffix"
        }
        
        # Case 3: Path is a directory link (no file extension)
        # We'll assume it's a directory if it doesn't have a file extension
        if ($path -ne "" -and $path -notmatch '\.[a-zA-Z0-9]+$') {
            # Remove trailing slash if present for consistency
            $cleanPath = $path -replace '/$', ''
            Write-Host "  Rewriting directory link: $path -> $repoUrl/$cleanPath"
            return "$prefix$repoUrl/$cleanPath$suffix"
        }
        
        # Default: return unchanged
        return $match.Value
    })
    
    # Only write the file if changes were made
    if ($newContent -ne $originalContent) {
        $matches = [regex]::Matches($content, $pattern).Where({
            $path = $_.Groups[2].Value
            # Count only the links we're actually rewriting
            return (
                ($path -match '^\.\./') -or 
                ($path -match '\.cs$') -or 
                ($path -ne "" -and $path -notmatch '\.[a-zA-Z0-9]+$' -and $path -notmatch '^(https?:|mailto:|#|javascript:)')
            )
        })
        
        $fileChanges = $matches.Count
        $linkCount += $fileChanges
        Set-Content -Path $file.FullName -Value $newContent
        Write-Host "Fixed $fileChanges links in $($file.Name)"
    }
}

Write-Host "Link transformation complete. Fixed $linkCount links in total."