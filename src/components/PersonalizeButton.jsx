// Clear preference
reset();

// Navigate back to base route
const pathParts = location.pathname.split('/');
const docsIndex = pathParts.indexOf('docs');

if (docsIndex !== -1 && pathParts.length > docsIndex + 2) {
  // Currently on personalized route, go back to base
  const chapterPath = pathParts[docsIndex + 1];
  const baseRoute = `/docs/${chapterPath}`;
  const fullPath = useBaseUrl(baseRoute);

  toast.success('Reset to original!', { duration: 2000 });
  history.push(fullPath);
} else {
  toast.success('Already on original content!', { duration: 2000 });
}
  };

return (
  <>
    <div style={{
      position: 'fixed',
      bottom: '80px',
      right: '20px',
      zIndex: 998,
    }}>
      {isActive ? (
        <button
          onClick={handleReset}
          className="navbar-signup-opts"
          style={{
            background: 'transparent',
            border: '2px solid #00d4ff',
            color: '#00d4ff',
            padding: '10px 20px',
            fontSize: '14px',
            fontWeight: 600,
            cursor: 'pointer',
            borderRadius: '8px',
            whiteSpace: 'nowrap',
          }}
        >
          ↩️ Reset to Original
        </button>
      ) : (
        <button
          onClick={() => setIsModalOpen(true)}
          className="navbar-signup-opts"
          style={{
            background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
            border: 'none',
            color: 'white',
            padding: '12px 24px',
            fontSize: '14px',
            fontWeight: 600,
            cursor: 'pointer',
            borderRadius: '8px',
            whiteSpace: 'nowrap',
            boxShadow: '0 4px 12px rgba(102, 126, 234, 0.4)',
          }}
        >
          ✨ Expert Mode
        </button>
      )}
    </div>

    <PersonalizationModal
      isOpen={isModalOpen}
      onClose={() => setIsModalOpen(false)}
      onSubmit={handlePersonalize}
    />
  </>
);
}
